/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "CG-ConfigInfo-v1610-IEs.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_measResultSCG_r16_constraint_5(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	if(1 /* No applicable constraints whatsoever */) {
		(void)st; /* Unused variable */
		/* Nothing is here. See below */
	}
	
	return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
}

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_sidelinkUEInformationNR_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	if(1 /* No applicable constraints whatsoever */) {
		(void)st; /* Unused variable */
		/* Nothing is here. See below */
	}
	
	return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_alignedDRX_Indication_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_failureType_r16_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_measResultSCG_r16_constr_15 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_SEMI_CONSTRAINED,	-1, -1,  0,  0 }	/* (SIZE(0..MAX)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_failureTypeEUTRA_r16_constr_17 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_sidelinkUEInformationNR_r16_constr_27 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_SEMI_CONSTRAINED,	-1, -1,  0,  0 }	/* (SIZE(0..MAX)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_alignedDRX_Indication_value2enum_3[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_alignedDRX_Indication_enum2value_3[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_alignedDRX_Indication_specs_3 = {
	asn_MAP_alignedDRX_Indication_value2enum_3,	/* "tag" => N; sorted by tag */
	asn_MAP_alignedDRX_Indication_enum2value_3,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_alignedDRX_Indication_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_alignedDRX_Indication_3 = {
	"alignedDRX-Indication",
	"alignedDRX-Indication",
	&asn_OP_NativeEnumerated,
	asn_DEF_alignedDRX_Indication_tags_3,
	sizeof(asn_DEF_alignedDRX_Indication_tags_3)
		/sizeof(asn_DEF_alignedDRX_Indication_tags_3[0]) - 1, /* 1 */
	asn_DEF_alignedDRX_Indication_tags_3,	/* Same as above */
	sizeof(asn_DEF_alignedDRX_Indication_tags_3)
		/sizeof(asn_DEF_alignedDRX_Indication_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_alignedDRX_Indication_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_alignedDRX_Indication_specs_3	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_failureType_r16_value2enum_6[] = {
	{ 0,	18,	"scg-lbtFailure-r16" },
	{ 1,	30,	"beamFailureRecoveryFailure-r16" },
	{ 2,	15,	"t312-Expiry-r16" },
	{ 3,	10,	"bh-RLF-r16" },
	{ 4,	15,	"beamFailure-r17" },
	{ 5,	6,	"spare3" },
	{ 6,	6,	"spare2" },
	{ 7,	6,	"spare1" }
};
static const unsigned int asn_MAP_failureType_r16_enum2value_6[] = {
	4,	/* beamFailure-r17(4) */
	1,	/* beamFailureRecoveryFailure-r16(1) */
	3,	/* bh-RLF-r16(3) */
	0,	/* scg-lbtFailure-r16(0) */
	7,	/* spare1(7) */
	6,	/* spare2(6) */
	5,	/* spare3(5) */
	2	/* t312-Expiry-r16(2) */
};
static const asn_INTEGER_specifics_t asn_SPC_failureType_r16_specs_6 = {
	asn_MAP_failureType_r16_value2enum_6,	/* "tag" => N; sorted by tag */
	asn_MAP_failureType_r16_enum2value_6,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_failureType_r16_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_failureType_r16_6 = {
	"failureType-r16",
	"failureType-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_failureType_r16_tags_6,
	sizeof(asn_DEF_failureType_r16_tags_6)
		/sizeof(asn_DEF_failureType_r16_tags_6[0]) - 1, /* 1 */
	asn_DEF_failureType_r16_tags_6,	/* Same as above */
	sizeof(asn_DEF_failureType_r16_tags_6)
		/sizeof(asn_DEF_failureType_r16_tags_6[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_failureType_r16_constr_6,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_failureType_r16_specs_6	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_scgFailureInfo_r16_5[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CG_ConfigInfo_v1610_IEs__scgFailureInfo_r16, failureType_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_failureType_r16_6,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"failureType-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CG_ConfigInfo_v1610_IEs__scgFailureInfo_r16, measResultSCG_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_measResultSCG_r16_constr_15,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_measResultSCG_r16_constraint_5
		},
		0, 0, /* No default value */
		"measResultSCG-r16"
		},
};
static const ber_tlv_tag_t asn_DEF_scgFailureInfo_r16_tags_5[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_scgFailureInfo_r16_tag2el_5[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* failureType-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* measResultSCG-r16 */
};
static asn_SEQUENCE_specifics_t asn_SPC_scgFailureInfo_r16_specs_5 = {
	sizeof(struct CG_ConfigInfo_v1610_IEs__scgFailureInfo_r16),
	offsetof(struct CG_ConfigInfo_v1610_IEs__scgFailureInfo_r16, _asn_ctx),
	asn_MAP_scgFailureInfo_r16_tag2el_5,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scgFailureInfo_r16_5 = {
	"scgFailureInfo-r16",
	"scgFailureInfo-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_scgFailureInfo_r16_tags_5,
	sizeof(asn_DEF_scgFailureInfo_r16_tags_5)
		/sizeof(asn_DEF_scgFailureInfo_r16_tags_5[0]) - 1, /* 1 */
	asn_DEF_scgFailureInfo_r16_tags_5,	/* Same as above */
	sizeof(asn_DEF_scgFailureInfo_r16_tags_5)
		/sizeof(asn_DEF_scgFailureInfo_r16_tags_5[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_scgFailureInfo_r16_5,
	2,	/* Elements count */
	&asn_SPC_scgFailureInfo_r16_specs_5	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_failureTypeEUTRA_r16_value2enum_17[] = {
	{ 0,	18,	"scg-lbtFailure-r16" },
	{ 1,	30,	"beamFailureRecoveryFailure-r16" },
	{ 2,	15,	"t312-Expiry-r16" },
	{ 3,	6,	"spare5" },
	{ 4,	6,	"spare4" },
	{ 5,	6,	"spare3" },
	{ 6,	6,	"spare2" },
	{ 7,	6,	"spare1" }
};
static const unsigned int asn_MAP_failureTypeEUTRA_r16_enum2value_17[] = {
	1,	/* beamFailureRecoveryFailure-r16(1) */
	0,	/* scg-lbtFailure-r16(0) */
	7,	/* spare1(7) */
	6,	/* spare2(6) */
	5,	/* spare3(5) */
	4,	/* spare4(4) */
	3,	/* spare5(3) */
	2	/* t312-Expiry-r16(2) */
};
static const asn_INTEGER_specifics_t asn_SPC_failureTypeEUTRA_r16_specs_17 = {
	asn_MAP_failureTypeEUTRA_r16_value2enum_17,	/* "tag" => N; sorted by tag */
	asn_MAP_failureTypeEUTRA_r16_enum2value_17,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_failureTypeEUTRA_r16_tags_17[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_failureTypeEUTRA_r16_17 = {
	"failureTypeEUTRA-r16",
	"failureTypeEUTRA-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_failureTypeEUTRA_r16_tags_17,
	sizeof(asn_DEF_failureTypeEUTRA_r16_tags_17)
		/sizeof(asn_DEF_failureTypeEUTRA_r16_tags_17[0]) - 1, /* 1 */
	asn_DEF_failureTypeEUTRA_r16_tags_17,	/* Same as above */
	sizeof(asn_DEF_failureTypeEUTRA_r16_tags_17)
		/sizeof(asn_DEF_failureTypeEUTRA_r16_tags_17[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_failureTypeEUTRA_r16_constr_17,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_failureTypeEUTRA_r16_specs_17	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_dummy1_16[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CG_ConfigInfo_v1610_IEs__dummy1, failureTypeEUTRA_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_failureTypeEUTRA_r16_17,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"failureTypeEUTRA-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CG_ConfigInfo_v1610_IEs__dummy1, measResultSCG_EUTRA_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"measResultSCG-EUTRA-r16"
		},
};
static const ber_tlv_tag_t asn_DEF_dummy1_tags_16[] = {
	(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_dummy1_tag2el_16[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* failureTypeEUTRA-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* measResultSCG-EUTRA-r16 */
};
static asn_SEQUENCE_specifics_t asn_SPC_dummy1_specs_16 = {
	sizeof(struct CG_ConfigInfo_v1610_IEs__dummy1),
	offsetof(struct CG_ConfigInfo_v1610_IEs__dummy1, _asn_ctx),
	asn_MAP_dummy1_tag2el_16,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_dummy1_16 = {
	"dummy1",
	"dummy1",
	&asn_OP_SEQUENCE,
	asn_DEF_dummy1_tags_16,
	sizeof(asn_DEF_dummy1_tags_16)
		/sizeof(asn_DEF_dummy1_tags_16[0]) - 1, /* 1 */
	asn_DEF_dummy1_tags_16,	/* Same as above */
	sizeof(asn_DEF_dummy1_tags_16)
		/sizeof(asn_DEF_dummy1_tags_16[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_dummy1_16,
	2,	/* Elements count */
	&asn_SPC_dummy1_specs_16	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_CG_ConfigInfo_v1610_IEs_1[] = {
	{ ATF_POINTER, 7, offsetof(struct CG_ConfigInfo_v1610_IEs, drx_InfoMCG2),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DRX_Info2,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"drx-InfoMCG2"
		},
	{ ATF_POINTER, 6, offsetof(struct CG_ConfigInfo_v1610_IEs, alignedDRX_Indication),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_alignedDRX_Indication_3,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"alignedDRX-Indication"
		},
	{ ATF_POINTER, 5, offsetof(struct CG_ConfigInfo_v1610_IEs, scgFailureInfo_r16),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		0,
		&asn_DEF_scgFailureInfo_r16_5,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"scgFailureInfo-r16"
		},
	{ ATF_POINTER, 4, offsetof(struct CG_ConfigInfo_v1610_IEs, dummy1),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		0,
		&asn_DEF_dummy1_16,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"dummy1"
		},
	{ ATF_POINTER, 3, offsetof(struct CG_ConfigInfo_v1610_IEs, sidelinkUEInformationNR_r16),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_sidelinkUEInformationNR_r16_constr_27,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_sidelinkUEInformationNR_r16_constraint_1
		},
		0, 0, /* No default value */
		"sidelinkUEInformationNR-r16"
		},
	{ ATF_POINTER, 2, offsetof(struct CG_ConfigInfo_v1610_IEs, sidelinkUEInformationEUTRA_r16),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"sidelinkUEInformationEUTRA-r16"
		},
	{ ATF_POINTER, 1, offsetof(struct CG_ConfigInfo_v1610_IEs, nonCriticalExtension),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CG_ConfigInfo_v1620_IEs,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"nonCriticalExtension"
		},
};
static const int asn_MAP_CG_ConfigInfo_v1610_IEs_oms_1[] = { 0, 1, 2, 3, 4, 5, 6 };
static const ber_tlv_tag_t asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CG_ConfigInfo_v1610_IEs_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* drx-InfoMCG2 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* alignedDRX-Indication */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* scgFailureInfo-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* dummy1 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* sidelinkUEInformationNR-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* sidelinkUEInformationEUTRA-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 } /* nonCriticalExtension */
};
asn_SEQUENCE_specifics_t asn_SPC_CG_ConfigInfo_v1610_IEs_specs_1 = {
	sizeof(struct CG_ConfigInfo_v1610_IEs),
	offsetof(struct CG_ConfigInfo_v1610_IEs, _asn_ctx),
	asn_MAP_CG_ConfigInfo_v1610_IEs_tag2el_1,
	7,	/* Count of tags in the map */
	asn_MAP_CG_ConfigInfo_v1610_IEs_oms_1,	/* Optional members */
	7, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CG_ConfigInfo_v1610_IEs = {
	"CG-ConfigInfo-v1610-IEs",
	"CG-ConfigInfo-v1610-IEs",
	&asn_OP_SEQUENCE,
	asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1,
	sizeof(asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1)
		/sizeof(asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1[0]), /* 1 */
	asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1,	/* Same as above */
	sizeof(asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1)
		/sizeof(asn_DEF_CG_ConfigInfo_v1610_IEs_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_CG_ConfigInfo_v1610_IEs_1,
	7,	/* Elements count */
	&asn_SPC_CG_ConfigInfo_v1610_IEs_specs_1	/* Additional specs */
};

