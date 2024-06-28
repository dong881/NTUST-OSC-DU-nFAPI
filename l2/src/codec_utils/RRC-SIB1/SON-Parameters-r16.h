/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SON_Parameters_r16_H_
#define	_SON_Parameters_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SON_Parameters_r16__rach_Report_r16 {
	SON_Parameters_r16__rach_Report_r16_supported	= 0
} e_SON_Parameters_r16__rach_Report_r16;
typedef enum SON_Parameters_r16__ext1__rlfReportCHO_r17 {
	SON_Parameters_r16__ext1__rlfReportCHO_r17_supported	= 0
} e_SON_Parameters_r16__ext1__rlfReportCHO_r17;
typedef enum SON_Parameters_r16__ext1__rlfReportDAPS_r17 {
	SON_Parameters_r16__ext1__rlfReportDAPS_r17_supported	= 0
} e_SON_Parameters_r16__ext1__rlfReportDAPS_r17;
typedef enum SON_Parameters_r16__ext1__success_HO_Report_r17 {
	SON_Parameters_r16__ext1__success_HO_Report_r17_supported	= 0
} e_SON_Parameters_r16__ext1__success_HO_Report_r17;
typedef enum SON_Parameters_r16__ext1__twoStepRACH_Report_r17 {
	SON_Parameters_r16__ext1__twoStepRACH_Report_r17_supported	= 0
} e_SON_Parameters_r16__ext1__twoStepRACH_Report_r17;
typedef enum SON_Parameters_r16__ext1__pscell_MHI_Report_r17 {
	SON_Parameters_r16__ext1__pscell_MHI_Report_r17_supported	= 0
} e_SON_Parameters_r16__ext1__pscell_MHI_Report_r17;
typedef enum SON_Parameters_r16__ext1__onDemandSI_Report_r17 {
	SON_Parameters_r16__ext1__onDemandSI_Report_r17_supported	= 0
} e_SON_Parameters_r16__ext1__onDemandSI_Report_r17;

/* SON-Parameters-r16 */
typedef struct SON_Parameters_r16 {
	long	*rach_Report_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SON_Parameters_r16__ext1 {
		long	*rlfReportCHO_r17;	/* OPTIONAL */
		long	*rlfReportDAPS_r17;	/* OPTIONAL */
		long	*success_HO_Report_r17;	/* OPTIONAL */
		long	*twoStepRACH_Report_r17;	/* OPTIONAL */
		long	*pscell_MHI_Report_r17;	/* OPTIONAL */
		long	*onDemandSI_Report_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SON_Parameters_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_rach_Report_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_rlfReportCHO_r17_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_rlfReportDAPS_r17_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_success_HO_Report_r17_10;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_twoStepRACH_Report_r17_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pscell_MHI_Report_r17_14;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_onDemandSI_Report_r17_16;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SON_Parameters_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SON_Parameters_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SON_Parameters_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SON_Parameters_r16_H_ */
#include <asn_internal.h>
