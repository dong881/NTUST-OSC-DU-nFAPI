/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "DAPS-UplinkPowerConfig-r16.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_uplinkPowerSharingDAPS_Mode_r16_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_uplinkPowerSharingDAPS_Mode_r16_value2enum_4[] = {
	{ 0,	17,	"semi-static-mode1" },
	{ 1,	17,	"semi-static-mode2" },
	{ 2,	7,	"dynamic" }
};
static const unsigned int asn_MAP_uplinkPowerSharingDAPS_Mode_r16_enum2value_4[] = {
	2,	/* dynamic(2) */
	0,	/* semi-static-mode1(0) */
	1	/* semi-static-mode2(1) */
};
static const asn_INTEGER_specifics_t asn_SPC_uplinkPowerSharingDAPS_Mode_r16_specs_4 = {
	asn_MAP_uplinkPowerSharingDAPS_Mode_r16_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_uplinkPowerSharingDAPS_Mode_r16_enum2value_4,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_uplinkPowerSharingDAPS_Mode_r16_4 = {
	"uplinkPowerSharingDAPS-Mode-r16",
	"uplinkPowerSharingDAPS-Mode-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4,
	sizeof(asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4)
		/sizeof(asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4[0]) - 1, /* 1 */
	asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4,	/* Same as above */
	sizeof(asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4)
		/sizeof(asn_DEF_uplinkPowerSharingDAPS_Mode_r16_tags_4[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_uplinkPowerSharingDAPS_Mode_r16_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_uplinkPowerSharingDAPS_Mode_r16_specs_4	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_DAPS_UplinkPowerConfig_r16_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct DAPS_UplinkPowerConfig_r16, p_DAPS_Source_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_P_Max,
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
		"p-DAPS-Source-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DAPS_UplinkPowerConfig_r16, p_DAPS_Target_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_P_Max,
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
		"p-DAPS-Target-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DAPS_UplinkPowerConfig_r16, uplinkPowerSharingDAPS_Mode_r16),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_uplinkPowerSharingDAPS_Mode_r16_4,
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
		"uplinkPowerSharingDAPS-Mode-r16"
		},
};
static const ber_tlv_tag_t asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DAPS_UplinkPowerConfig_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* p-DAPS-Source-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* p-DAPS-Target-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* uplinkPowerSharingDAPS-Mode-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_DAPS_UplinkPowerConfig_r16_specs_1 = {
	sizeof(struct DAPS_UplinkPowerConfig_r16),
	offsetof(struct DAPS_UplinkPowerConfig_r16, _asn_ctx),
	asn_MAP_DAPS_UplinkPowerConfig_r16_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_DAPS_UplinkPowerConfig_r16 = {
	"DAPS-UplinkPowerConfig-r16",
	"DAPS-UplinkPowerConfig-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1,
	sizeof(asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1)
		/sizeof(asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1[0]), /* 1 */
	asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1)
		/sizeof(asn_DEF_DAPS_UplinkPowerConfig_r16_tags_1[0]), /* 1 */
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
	asn_MBR_DAPS_UplinkPowerConfig_r16_1,
	3,	/* Elements count */
	&asn_SPC_DAPS_UplinkPowerConfig_r16_specs_1	/* Additional specs */
};

