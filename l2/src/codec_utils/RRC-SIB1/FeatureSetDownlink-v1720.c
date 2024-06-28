/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "FeatureSetDownlink-v1720.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_rtt_BasedPDC_CSI_RS_ForTracking_r17_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_maxNumberPRS_Resource_r17_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  6 }	/* (0..6) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs_15kHz_r17_constr_14 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  10 }	/* (0..10) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs_30kHz_r17_constr_26 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  10 }	/* (0..10) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs_60kHz_r17_constr_38 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  10 }	/* (0..10) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs_120kHz_r17_constr_50 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  10 }	/* (0..10) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sps_Multicast_r17_constr_62 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_rtt_BasedPDC_CSI_RS_ForTracking_r17_value2enum_2[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_rtt_BasedPDC_CSI_RS_ForTracking_r17_enum2value_2[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_rtt_BasedPDC_CSI_RS_ForTracking_r17_specs_2 = {
	asn_MAP_rtt_BasedPDC_CSI_RS_ForTracking_r17_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_rtt_BasedPDC_CSI_RS_ForTracking_r17_enum2value_2,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_2 = {
	"rtt-BasedPDC-CSI-RS-ForTracking-r17",
	"rtt-BasedPDC-CSI-RS-ForTracking-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2,
	sizeof(asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2)
		/sizeof(asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2)
		/sizeof(asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_rtt_BasedPDC_CSI_RS_ForTracking_r17_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_rtt_BasedPDC_CSI_RS_ForTracking_r17_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_maxNumberPRS_Resource_r17_value2enum_5[] = {
	{ 0,	2,	"n1" },
	{ 1,	2,	"n2" },
	{ 2,	2,	"n4" },
	{ 3,	2,	"n8" },
	{ 4,	3,	"n16" },
	{ 5,	3,	"n32" },
	{ 6,	3,	"n64" }
};
static const unsigned int asn_MAP_maxNumberPRS_Resource_r17_enum2value_5[] = {
	0,	/* n1(0) */
	4,	/* n16(4) */
	1,	/* n2(1) */
	5,	/* n32(5) */
	2,	/* n4(2) */
	6,	/* n64(6) */
	3	/* n8(3) */
};
static const asn_INTEGER_specifics_t asn_SPC_maxNumberPRS_Resource_r17_specs_5 = {
	asn_MAP_maxNumberPRS_Resource_r17_value2enum_5,	/* "tag" => N; sorted by tag */
	asn_MAP_maxNumberPRS_Resource_r17_enum2value_5,	/* N => "tag"; sorted by N */
	7,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_maxNumberPRS_Resource_r17_tags_5[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxNumberPRS_Resource_r17_5 = {
	"maxNumberPRS-Resource-r17",
	"maxNumberPRS-Resource-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_maxNumberPRS_Resource_r17_tags_5,
	sizeof(asn_DEF_maxNumberPRS_Resource_r17_tags_5)
		/sizeof(asn_DEF_maxNumberPRS_Resource_r17_tags_5[0]) - 1, /* 1 */
	asn_DEF_maxNumberPRS_Resource_r17_tags_5,	/* Same as above */
	sizeof(asn_DEF_maxNumberPRS_Resource_r17_tags_5)
		/sizeof(asn_DEF_maxNumberPRS_Resource_r17_tags_5[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_maxNumberPRS_Resource_r17_constr_5,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_maxNumberPRS_Resource_r17_specs_5	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_scs_15kHz_r17_value2enum_14[] = {
	{ 0,	2,	"n1" },
	{ 1,	2,	"n2" },
	{ 2,	2,	"n4" },
	{ 3,	2,	"n6" },
	{ 4,	2,	"n8" },
	{ 5,	3,	"n12" },
	{ 6,	3,	"n16" },
	{ 7,	3,	"n24" },
	{ 8,	3,	"n32" },
	{ 9,	3,	"n48" },
	{ 10,	3,	"n64" }
};
static const unsigned int asn_MAP_scs_15kHz_r17_enum2value_14[] = {
	0,	/* n1(0) */
	5,	/* n12(5) */
	6,	/* n16(6) */
	1,	/* n2(1) */
	7,	/* n24(7) */
	8,	/* n32(8) */
	2,	/* n4(2) */
	9,	/* n48(9) */
	3,	/* n6(3) */
	10,	/* n64(10) */
	4	/* n8(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs_15kHz_r17_specs_14 = {
	asn_MAP_scs_15kHz_r17_value2enum_14,	/* "tag" => N; sorted by tag */
	asn_MAP_scs_15kHz_r17_enum2value_14,	/* N => "tag"; sorted by N */
	11,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs_15kHz_r17_tags_14[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs_15kHz_r17_14 = {
	"scs-15kHz-r17",
	"scs-15kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs_15kHz_r17_tags_14,
	sizeof(asn_DEF_scs_15kHz_r17_tags_14)
		/sizeof(asn_DEF_scs_15kHz_r17_tags_14[0]) - 1, /* 1 */
	asn_DEF_scs_15kHz_r17_tags_14,	/* Same as above */
	sizeof(asn_DEF_scs_15kHz_r17_tags_14)
		/sizeof(asn_DEF_scs_15kHz_r17_tags_14[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs_15kHz_r17_constr_14,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs_15kHz_r17_specs_14	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_scs_30kHz_r17_value2enum_26[] = {
	{ 0,	2,	"n1" },
	{ 1,	2,	"n2" },
	{ 2,	2,	"n4" },
	{ 3,	2,	"n6" },
	{ 4,	2,	"n8" },
	{ 5,	3,	"n12" },
	{ 6,	3,	"n16" },
	{ 7,	3,	"n24" },
	{ 8,	3,	"n32" },
	{ 9,	3,	"n48" },
	{ 10,	3,	"n64" }
};
static const unsigned int asn_MAP_scs_30kHz_r17_enum2value_26[] = {
	0,	/* n1(0) */
	5,	/* n12(5) */
	6,	/* n16(6) */
	1,	/* n2(1) */
	7,	/* n24(7) */
	8,	/* n32(8) */
	2,	/* n4(2) */
	9,	/* n48(9) */
	3,	/* n6(3) */
	10,	/* n64(10) */
	4	/* n8(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs_30kHz_r17_specs_26 = {
	asn_MAP_scs_30kHz_r17_value2enum_26,	/* "tag" => N; sorted by tag */
	asn_MAP_scs_30kHz_r17_enum2value_26,	/* N => "tag"; sorted by N */
	11,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs_30kHz_r17_tags_26[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs_30kHz_r17_26 = {
	"scs-30kHz-r17",
	"scs-30kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs_30kHz_r17_tags_26,
	sizeof(asn_DEF_scs_30kHz_r17_tags_26)
		/sizeof(asn_DEF_scs_30kHz_r17_tags_26[0]) - 1, /* 1 */
	asn_DEF_scs_30kHz_r17_tags_26,	/* Same as above */
	sizeof(asn_DEF_scs_30kHz_r17_tags_26)
		/sizeof(asn_DEF_scs_30kHz_r17_tags_26[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs_30kHz_r17_constr_26,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs_30kHz_r17_specs_26	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_scs_60kHz_r17_value2enum_38[] = {
	{ 0,	2,	"n1" },
	{ 1,	2,	"n2" },
	{ 2,	2,	"n4" },
	{ 3,	2,	"n6" },
	{ 4,	2,	"n8" },
	{ 5,	3,	"n12" },
	{ 6,	3,	"n16" },
	{ 7,	3,	"n24" },
	{ 8,	3,	"n32" },
	{ 9,	3,	"n48" },
	{ 10,	3,	"n64" }
};
static const unsigned int asn_MAP_scs_60kHz_r17_enum2value_38[] = {
	0,	/* n1(0) */
	5,	/* n12(5) */
	6,	/* n16(6) */
	1,	/* n2(1) */
	7,	/* n24(7) */
	8,	/* n32(8) */
	2,	/* n4(2) */
	9,	/* n48(9) */
	3,	/* n6(3) */
	10,	/* n64(10) */
	4	/* n8(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs_60kHz_r17_specs_38 = {
	asn_MAP_scs_60kHz_r17_value2enum_38,	/* "tag" => N; sorted by tag */
	asn_MAP_scs_60kHz_r17_enum2value_38,	/* N => "tag"; sorted by N */
	11,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs_60kHz_r17_tags_38[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs_60kHz_r17_38 = {
	"scs-60kHz-r17",
	"scs-60kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs_60kHz_r17_tags_38,
	sizeof(asn_DEF_scs_60kHz_r17_tags_38)
		/sizeof(asn_DEF_scs_60kHz_r17_tags_38[0]) - 1, /* 1 */
	asn_DEF_scs_60kHz_r17_tags_38,	/* Same as above */
	sizeof(asn_DEF_scs_60kHz_r17_tags_38)
		/sizeof(asn_DEF_scs_60kHz_r17_tags_38[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs_60kHz_r17_constr_38,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs_60kHz_r17_specs_38	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_scs_120kHz_r17_value2enum_50[] = {
	{ 0,	2,	"n1" },
	{ 1,	2,	"n2" },
	{ 2,	2,	"n4" },
	{ 3,	2,	"n6" },
	{ 4,	2,	"n8" },
	{ 5,	3,	"n12" },
	{ 6,	3,	"n16" },
	{ 7,	3,	"n24" },
	{ 8,	3,	"n32" },
	{ 9,	3,	"n48" },
	{ 10,	3,	"n64" }
};
static const unsigned int asn_MAP_scs_120kHz_r17_enum2value_50[] = {
	0,	/* n1(0) */
	5,	/* n12(5) */
	6,	/* n16(6) */
	1,	/* n2(1) */
	7,	/* n24(7) */
	8,	/* n32(8) */
	2,	/* n4(2) */
	9,	/* n48(9) */
	3,	/* n6(3) */
	10,	/* n64(10) */
	4	/* n8(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs_120kHz_r17_specs_50 = {
	asn_MAP_scs_120kHz_r17_value2enum_50,	/* "tag" => N; sorted by tag */
	asn_MAP_scs_120kHz_r17_enum2value_50,	/* N => "tag"; sorted by N */
	11,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs_120kHz_r17_tags_50[] = {
	(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs_120kHz_r17_50 = {
	"scs-120kHz-r17",
	"scs-120kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs_120kHz_r17_tags_50,
	sizeof(asn_DEF_scs_120kHz_r17_tags_50)
		/sizeof(asn_DEF_scs_120kHz_r17_tags_50[0]) - 1, /* 1 */
	asn_DEF_scs_120kHz_r17_tags_50,	/* Same as above */
	sizeof(asn_DEF_scs_120kHz_r17_tags_50)
		/sizeof(asn_DEF_scs_120kHz_r17_tags_50[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs_120kHz_r17_constr_50,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs_120kHz_r17_specs_50	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_maxNumberPRS_ResourceProcessedPerSlot_r17_13[] = {
	{ ATF_POINTER, 4, offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17__maxNumberPRS_ResourceProcessedPerSlot_r17, scs_15kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs_15kHz_r17_14,
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
		"scs-15kHz-r17"
		},
	{ ATF_POINTER, 3, offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17__maxNumberPRS_ResourceProcessedPerSlot_r17, scs_30kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs_30kHz_r17_26,
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
		"scs-30kHz-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17__maxNumberPRS_ResourceProcessedPerSlot_r17, scs_60kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs_60kHz_r17_38,
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
		"scs-60kHz-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17__maxNumberPRS_ResourceProcessedPerSlot_r17, scs_120kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs_120kHz_r17_50,
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
		"scs-120kHz-r17"
		},
};
static const int asn_MAP_maxNumberPRS_ResourceProcessedPerSlot_r17_oms_13[] = { 0, 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_maxNumberPRS_ResourceProcessedPerSlot_r17_tag2el_13[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* scs-15kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* scs-30kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* scs-60kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* scs-120kHz-r17 */
};
static asn_SEQUENCE_specifics_t asn_SPC_maxNumberPRS_ResourceProcessedPerSlot_r17_specs_13 = {
	sizeof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17__maxNumberPRS_ResourceProcessedPerSlot_r17),
	offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17__maxNumberPRS_ResourceProcessedPerSlot_r17, _asn_ctx),
	asn_MAP_maxNumberPRS_ResourceProcessedPerSlot_r17_tag2el_13,
	4,	/* Count of tags in the map */
	asn_MAP_maxNumberPRS_ResourceProcessedPerSlot_r17_oms_13,	/* Optional members */
	4, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_13 = {
	"maxNumberPRS-ResourceProcessedPerSlot-r17",
	"maxNumberPRS-ResourceProcessedPerSlot-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13,
	sizeof(asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13)
		/sizeof(asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13[0]) - 1, /* 1 */
	asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13,	/* Same as above */
	sizeof(asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13)
		/sizeof(asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_tags_13[0]), /* 2 */
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
	asn_MBR_maxNumberPRS_ResourceProcessedPerSlot_r17_13,
	4,	/* Elements count */
	&asn_SPC_maxNumberPRS_ResourceProcessedPerSlot_r17_specs_13	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_rtt_BasedPDC_PRS_r17_4[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17, maxNumberPRS_Resource_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_maxNumberPRS_Resource_r17_5,
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
		"maxNumberPRS-Resource-r17"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17, maxNumberPRS_ResourceProcessedPerSlot_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_maxNumberPRS_ResourceProcessedPerSlot_r17_13,
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
		"maxNumberPRS-ResourceProcessedPerSlot-r17"
		},
};
static const ber_tlv_tag_t asn_DEF_rtt_BasedPDC_PRS_r17_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_rtt_BasedPDC_PRS_r17_tag2el_4[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* maxNumberPRS-Resource-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* maxNumberPRS-ResourceProcessedPerSlot-r17 */
};
static asn_SEQUENCE_specifics_t asn_SPC_rtt_BasedPDC_PRS_r17_specs_4 = {
	sizeof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17),
	offsetof(struct FeatureSetDownlink_v1720__rtt_BasedPDC_PRS_r17, _asn_ctx),
	asn_MAP_rtt_BasedPDC_PRS_r17_tag2el_4,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_rtt_BasedPDC_PRS_r17_4 = {
	"rtt-BasedPDC-PRS-r17",
	"rtt-BasedPDC-PRS-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_rtt_BasedPDC_PRS_r17_tags_4,
	sizeof(asn_DEF_rtt_BasedPDC_PRS_r17_tags_4)
		/sizeof(asn_DEF_rtt_BasedPDC_PRS_r17_tags_4[0]) - 1, /* 1 */
	asn_DEF_rtt_BasedPDC_PRS_r17_tags_4,	/* Same as above */
	sizeof(asn_DEF_rtt_BasedPDC_PRS_r17_tags_4)
		/sizeof(asn_DEF_rtt_BasedPDC_PRS_r17_tags_4[0]), /* 2 */
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
	asn_MBR_rtt_BasedPDC_PRS_r17_4,
	2,	/* Elements count */
	&asn_SPC_rtt_BasedPDC_PRS_r17_specs_4	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_sps_Multicast_r17_value2enum_62[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_sps_Multicast_r17_enum2value_62[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_sps_Multicast_r17_specs_62 = {
	asn_MAP_sps_Multicast_r17_value2enum_62,	/* "tag" => N; sorted by tag */
	asn_MAP_sps_Multicast_r17_enum2value_62,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_sps_Multicast_r17_tags_62[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sps_Multicast_r17_62 = {
	"sps-Multicast-r17",
	"sps-Multicast-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_sps_Multicast_r17_tags_62,
	sizeof(asn_DEF_sps_Multicast_r17_tags_62)
		/sizeof(asn_DEF_sps_Multicast_r17_tags_62[0]) - 1, /* 1 */
	asn_DEF_sps_Multicast_r17_tags_62,	/* Same as above */
	sizeof(asn_DEF_sps_Multicast_r17_tags_62)
		/sizeof(asn_DEF_sps_Multicast_r17_tags_62[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sps_Multicast_r17_constr_62,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_sps_Multicast_r17_specs_62	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_FeatureSetDownlink_v1720_1[] = {
	{ ATF_POINTER, 3, offsetof(struct FeatureSetDownlink_v1720, rtt_BasedPDC_CSI_RS_ForTracking_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_rtt_BasedPDC_CSI_RS_ForTracking_r17_2,
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
		"rtt-BasedPDC-CSI-RS-ForTracking-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct FeatureSetDownlink_v1720, rtt_BasedPDC_PRS_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_rtt_BasedPDC_PRS_r17_4,
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
		"rtt-BasedPDC-PRS-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct FeatureSetDownlink_v1720, sps_Multicast_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_sps_Multicast_r17_62,
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
		"sps-Multicast-r17"
		},
};
static const int asn_MAP_FeatureSetDownlink_v1720_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_FeatureSetDownlink_v1720_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_FeatureSetDownlink_v1720_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* rtt-BasedPDC-CSI-RS-ForTracking-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* rtt-BasedPDC-PRS-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* sps-Multicast-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_FeatureSetDownlink_v1720_specs_1 = {
	sizeof(struct FeatureSetDownlink_v1720),
	offsetof(struct FeatureSetDownlink_v1720, _asn_ctx),
	asn_MAP_FeatureSetDownlink_v1720_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_FeatureSetDownlink_v1720_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_FeatureSetDownlink_v1720 = {
	"FeatureSetDownlink-v1720",
	"FeatureSetDownlink-v1720",
	&asn_OP_SEQUENCE,
	asn_DEF_FeatureSetDownlink_v1720_tags_1,
	sizeof(asn_DEF_FeatureSetDownlink_v1720_tags_1)
		/sizeof(asn_DEF_FeatureSetDownlink_v1720_tags_1[0]), /* 1 */
	asn_DEF_FeatureSetDownlink_v1720_tags_1,	/* Same as above */
	sizeof(asn_DEF_FeatureSetDownlink_v1720_tags_1)
		/sizeof(asn_DEF_FeatureSetDownlink_v1720_tags_1[0]), /* 1 */
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
	asn_MBR_FeatureSetDownlink_v1720_1,
	3,	/* Elements count */
	&asn_SPC_FeatureSetDownlink_v1720_specs_1	/* Additional specs */
};

